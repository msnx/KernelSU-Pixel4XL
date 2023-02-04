/*
 * NIAP FPT_TST_EXT.1 cryptographic self-tests.
 *
 * Copyright (c) 2019 Google LLC.
 */

#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include <linux/crypto.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/scatterlist.h>

static const u8 niap_message[] __initconst = {
	0x6d, 0x65, 0x73, 0x73, 0x61, 0x67, 0x65, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const u8 niap_hmac_sha512_key[] __initconst = {
	0x84, 0xe1, 0xb6, 0x59, 0x69, 0x4f, 0x5e, 0xa5,
	0x07, 0xcc, 0x74, 0x06, 0xb8, 0xec, 0x53, 0x4a
};

static const u8 niap_hmac_sha512_output[] __initconst = {
	0x72, 0x74, 0x73, 0x5e, 0xa5, 0x1d, 0x1c, 0xba,
	0x62, 0xa7, 0x93, 0xbb, 0xa9, 0xc3, 0x97, 0x86,
	0x9d, 0x46, 0x26, 0xe7, 0x1b, 0x8c, 0x73, 0xf7,
	0x1a, 0xcb, 0xae, 0xda, 0x32, 0x77, 0x2d, 0x9b,
	0x73, 0xa1, 0xb1, 0x21, 0x16, 0x00, 0x38, 0xc8,
	0x61, 0x9c, 0xb9, 0x7f, 0x3c, 0x3b, 0x16, 0xe1,
	0xb8, 0x1d, 0x34, 0x8c, 0x82, 0xdc, 0xe6, 0xe3,
	0xc6, 0xaa, 0x4c, 0x10, 0x54, 0x3c, 0x09, 0xf4
};

static const u8 niap_sha256_output[] __initconst = {
	0xa0, 0x90, 0x1c, 0xb8, 0xb4, 0x22, 0xa3, 0xef,
	0x7d, 0x34, 0x45, 0x2d, 0x73, 0x32, 0x6c, 0x1e,
	0x09, 0x2d, 0x9e, 0x82, 0xee, 0x3f, 0x32, 0xd3,
	0xe8, 0x64, 0x30, 0x50, 0xb0, 0x2e, 0x7d, 0xf4
};

static const u8 niap_cbc_aes128_encrypt_key[] __initconst = {
	0x6f, 0x1e, 0x97, 0x12, 0xcf, 0x82, 0x40, 0x79,
	0x2c, 0x4f, 0x8a, 0x82, 0x78, 0x2a, 0x8c, 0xdd
};

static const u8 niap_cbc_aes128_decrypt_key[] __initconst = {
	0x02, 0x52, 0x38, 0x55, 0xa6, 0x88, 0x51, 0xe2,
	0xd0, 0x16, 0xaa, 0x56, 0x95, 0x64, 0x33, 0x2d
};

static const u8 niap_cbc_aes128_iv[] __initconst = {
	0x0c, 0xa7, 0x8b, 0xd2, 0x55, 0x73, 0x85, 0xef,
	0xe2, 0x1b, 0x96, 0xb1, 0x0f, 0x14, 0xec, 0xf0
};

static const u8 niap_cbc_aes128_encrypt_output[] __initconst = {
	0xd1, 0x4f, 0xe0, 0x79, 0xa1, 0x68, 0xe4, 0xa4,
	0x51, 0x96, 0xf6, 0x10, 0x5d, 0x6a, 0x63, 0xa0
};

static const u8 niap_cbc_aes128_decrypt_output[] __initconst = {
	0x59, 0x39, 0x9b, 0xb7, 0x7d, 0xef, 0x86, 0x7b,
	0x0d, 0xe0, 0x77, 0x90, 0x17, 0x7c, 0x2a, 0xdb
};

struct niap_test {
	const char *alg;
	const u8 *key;
	unsigned int key_length;
	const u8 *iv;
	unsigned int iv_length;
	const u8 *input;
	unsigned int in_length;
	const u8 *output;
	unsigned int out_length;
	bool encrypt;
	int (*func)(const struct niap_test *);
};

static inline void dump(const u8 *buf, unsigned int length)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 1, buf, length,
		false);
}

static int __init niap_test_cipher(const struct niap_test *test)
{
	int err = 0;
	DECLARE_CRYPTO_WAIT(wait);
	struct crypto_skcipher *tfm = NULL;
	struct skcipher_request *req = NULL;
	struct scatterlist src;
	struct scatterlist dst;
	u8 *input = NULL;
	u8 *iv = NULL;
	u8 *output = NULL;

	input	= kmemdup(test->input, test->in_length,  GFP_KERNEL);
	iv	= kmemdup(test->iv, test->iv_length,  GFP_KERNEL);
	output	= kzalloc(test->out_length, GFP_KERNEL);

	if (!input || !iv || !output) {
		err = -ENOMEM;
		goto out;
	}

	tfm = crypto_alloc_skcipher(test->alg, 0, 0);
	if (IS_ERR(tfm)) {
		err = PTR_ERR(tfm);
		tfm = NULL;
		goto out;
	}

	err = crypto_skcipher_setkey(tfm, test->key, test->key_length);
	if (err)
		goto out;

	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		err = -ENOMEM;
		goto out;
	}

	sg_init_one(&src, input, test->in_length);
	sg_init_one(&dst, output, test->out_length);

	skcipher_request_set_tfm(req, tfm);
	skcipher_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_SLEEP | CRYPTO_TFM_REQ_MAY_BACKLOG,
		crypto_req_done, &wait);
	skcipher_request_set_crypt(req, &src, &dst, test->in_length, iv);

	if (test->encrypt)
		err = crypto_wait_req(crypto_skcipher_encrypt(req), &wait);
	else
		err = crypto_wait_req(crypto_skcipher_decrypt(req), &wait);
	if (err)
		goto out;

	if (memcmp(output, test->output, test->out_length)) {
		pr_err("niap_test_cipher: invalid result:\n");
		dump(output, test->out_length);
		err = -EBADMSG;
	}

out:
	if (err)
		pr_err("niap_test_cipher: %s %s failed: %d\n", test->alg,
			test->encrypt ? "encryption" : "decryption", err);
	else
		pr_info("niap_test_cipher: %s %s passed\n", test->alg,
			test->encrypt ? "encryption" : "decryption");

	skcipher_request_free(req);
	crypto_free_skcipher(tfm);
	kfree(input);
	kfree(iv);
	kfree(output);

	return err;
}

static int __init niap_test_hash(const struct niap_test *test)
{
	int err = 0;
	struct crypto_shash *tfm = NULL;
	struct shash_desc *desc = NULL;
	u8 *output = NULL;

	output = kzalloc(test->out_length, GFP_KERNEL);
	if (!output) {
		err = -ENOMEM;
		goto out;
	}

	tfm = crypto_alloc_shash(test->alg, 0, 0);
	if (IS_ERR(tfm)) {
		err = PTR_ERR(tfm);
		tfm = NULL;
		goto out;
	}

	if (test->key) {
		err = crypto_shash_setkey(tfm, test->key, test->key_length);
		if (err)
			goto out;
	}

	desc = kzalloc(sizeof(*desc) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (!desc) {
		err = -ENOMEM;
		goto out;
	}

	desc->tfm = tfm;
	desc->flags = CRYPTO_TFM_REQ_MAY_SLEEP;

	err = crypto_shash_digest(desc, test->input, test->in_length, output);
	if (err)
		goto out;

	if (memcmp(output, test->output, test->out_length)) {
		pr_err("niap_test_hash: invalid result:\n");
		dump(output, test->out_length);
		err = -EBADMSG;
	}

out:
	if (err)
		pr_err("niap_test_hash: %s failed: %d\n", test->alg, err);
	else
		pr_info("niap_test_hash: %s passed\n", test->alg);

	crypto_free_shash(tfm);
	kfree(desc);
	kfree(output);

	return err;
}

static const struct niap_test niap_tests[] __initconst = {
	{
		.alg		= "hmac(sha512)",
		.key		= niap_hmac_sha512_key,
		.key_length	= sizeof(niap_hmac_sha512_key),
		.input		= niap_message,
		.in_length	= sizeof(niap_message),
		.output		= niap_hmac_sha512_output,
		.out_length	= sizeof(niap_hmac_sha512_output),
		.func		= niap_test_hash,
	}, {
		.alg		= "sha256",
		.key		= NULL,
		.input		= niap_message,
		.in_length	= sizeof(niap_message),
		.output		= niap_sha256_output,
		.out_length	= sizeof(niap_sha256_output),
		.func		= niap_test_hash,
	}, {
		.alg		= "cbc(aes)",
		.key		= niap_cbc_aes128_encrypt_key,
		.key_length	= sizeof(niap_cbc_aes128_encrypt_key),
		.input		= niap_message,
		.in_length	= sizeof(niap_message),
		.iv		= niap_cbc_aes128_iv,
		.iv_length	= sizeof(niap_cbc_aes128_iv),
		.output		= niap_cbc_aes128_encrypt_output,
		.out_length	= sizeof(niap_cbc_aes128_encrypt_output),
		.encrypt	= true,
		.func		= niap_test_cipher,
	}, {
		.alg		= "cbc(aes)",
		.key		= niap_cbc_aes128_decrypt_key,
		.key_length	= sizeof(niap_cbc_aes128_decrypt_key),
		.input		= niap_message,
		.in_length	= sizeof(niap_message),
		.iv		= niap_cbc_aes128_iv,
		.iv_length	= sizeof(niap_cbc_aes128_iv),
		.output		= niap_cbc_aes128_decrypt_output,
		.out_length	= sizeof(niap_cbc_aes128_decrypt_output),
		.encrypt	= false,
		.func		= niap_test_cipher,
	}
};

static int __init niap_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(niap_tests); i++) {
		const struct niap_test *test = &niap_tests[i];

		if (test->func(test)) {
			pr_crit("niap: self-test failed; rebooting\n");
			kernel_restart("bootloader");
		}
	}

	return 0;
}

late_initcall(niap_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NIAP FPT_TST_EXT.1 cryptographic self-tests");
